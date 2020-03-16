"use strict";

import React from 'react';
// import { DataTable, TableHeader, TableBody, TableRow, TableColumn, Button } from 'react-md';
import {
    Badge,
    Button,
    ButtonDropdown,
    ButtonGroup,
    ButtonToolbar,
    Card,
    CardBody,
    CardFooter,
    CardHeader,
    CardTitle,
    Col,
    Dropdown,
    DropdownItem,
    FormText,
    FormGroup,
    Form,
    Row,
    Table,
} from 'reactstrap';
import { Link } from 'react-router-dom';
import { BookingListRow } from './BookingListRow';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { ToastContainer, toast } from 'react-toastify';
import Page from './Page'


export class BookingList extends React.Component {

    constructor(props) {
        super(props);
        console.log('here I am',props);
        this.state = {
            loading: false,
            bookings: [...props.data]
        };
        this.onClickHandler = this.onClickHandler.bind(this);
    }//= ({data, itemId, onDelete}) => (

    onClickHandler(id){
        this.props.onClick(id);
    }
    render(){
        let bookings = this.state.bookings;
        console.log('bookings', bookings);
        let self = this;
        // console.log(new Intl.DateTimeFormat('en-US', {year: 'numeric', month: '2-digit',day: '2-digit', hour: '2-digit', minute: '2-digit', second: '2-digit'}).format(bookings[0].endDate));
        if(this.props.match.path === '/my_item_bookings/:id'){
            return(
                <Page>
                    <div className="animated fadeIn">
                        <Row >
                            <Col className={'md'}>
                                <Card >
                                    <CardHeader>
                                        <b>{bookings[0].item_id.name}</b> Bookings
                                    </CardHeader>
                                    <CardBody>

                                        <Table hover responsive className="table-outline mb-0 d-none d-table">
                                            <thead className="thead-light">
                                            <tr>
                                                <th className="text-center"><FontAwesomeIcon icon="users" /></th>
                                                <th className="text-left">Username</th>
                                                <th className="text-center">Start Date</th>
                                                <th className="text-center">End Date</th>
                                                <th className="text-center">View Booking Details</th>

                                            </tr>
                                            </thead>

                                            <tbody>
                                            {bookings.map((booking, i) => <BookingListRow key={i} itemId= {this.props.itemId} booking={booking} onClick={(id) => self.onClickHandler(id)} />)}

                                            </tbody>
                                        </Table>
                                    </CardBody>
                                </Card>
                            </Col>
                        </Row>
                    </div>
                </Page>
            );
        }
        else{
            return(
                <Page>
                    <div className="animated fadeIn">
                        <Row >
                            <Col className={'md'}>
                                <Card >
                                    <CardHeader>
                                        <b>My Bookings</b>
                                        {/*<b>{bookings[0].booker_id.name}</b> Bookings*/}
                                    </CardHeader>
                                    <CardBody>

                                        <Table hover responsive className="table-outline mb-0 d-none d-table">
                                            <thead className="thead-light">
                                            <tr>
                                                <th className="text-center"><FontAwesomeIcon icon="users" /></th>
                                                <th className="text-left">Username</th>
                                                <th className="text-center">Start Date</th>
                                                <th className="text-center">End Date</th>
                                                <th className="text-center">View Booking Details</th>

                                            </tr>
                                            </thead>

                                            <tbody>
                                            {bookings.map((booking, i) => <BookingListRow key={i} itemId= {this.props.itemId} booking={booking} onClick={(id) => self.onClickHandler(id)} />)}

                                            </tbody>
                                        </Table>
                                    </CardBody>
                                </Card>
                            </Col>
                        </Row>
                    </div>
                </Page>
            );

        }
    }
}



    // <Page>
    //     <b><font size="5" face="verdana" color="black">Your bookings</font></b>
    //     <DataTable plain>
    //         <TableHeader>
    //             <TableRow>
    //                 <TableColumn></TableColumn>
    //                 <TableColumn>Name</TableColumn>
    //                 <TableColumn>Edit</TableColumn>
    //                 <TableColumn>Remove</TableColumn>
    //                 <TableColumn>Evaluate</TableColumn>
    //
    //             </TableRow>
    //         </TableHeader>
    //         <TableBody>
    //             {data.map((booking, i) => <BookingListRow key={i} itemId= {itemId} booking={booking} onDelete={(id) => onDelete(id)} />)}
    //         </TableBody>
    //     </DataTable>
    // </Page>


